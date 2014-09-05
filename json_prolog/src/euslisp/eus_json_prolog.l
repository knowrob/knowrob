;;
;; json_prolog interface for euslisp
;;
;; Euslisp and roseus is needed for using this interface.
;; Please download these packages from
;;  https://jsk-ros-pkg.svn.sourceforge.net/svnroot/jsk-ros-pkg/trunk
;;
;; Author: Manabu Saito <saito@jsk.t.u-tokyo.ac.jp>
;;

(ros::roseus-add-srvs "json_prolog_msgs")

;; convert json to list, this can't convert lists correctly
(defun read-json (str)
  (let ((*readtable* (copy-readtable)) ret item items p)
;    (set-syntax-from-char #\[ #\() ;; array
;    (set-syntax-from-char #\] #\)) ;; array
	(set-macro-character #\[ #'(lambda(s c)(read-delimited-list #\] s t)))
	(set-macro-character #\] (get-macro-character #\)))
;    (set-syntax-from-char #\{ #\() ;; list
;    (set-syntax-from-char #\} #\)) ;; list
	(set-macro-character #\{ #'(lambda(s c)(read-delimited-list #\} s t)))
	(set-macro-character #\} (get-macro-character #\)))

    (set-syntax-from-char #\, #\ ) ;;
	(set-macro-character #\: #'(lambda (s c) :json-colon))

	(setq ret (read-from-string str))
	(while (setq p (position :json-colon ret))
	  (setq ret (append (subseq ret 0 (1- p))
						(list (list (elt ret (1- p)) (elt ret (1+ p))))
						(if (< (+ 2 p) (length ret)) (subseq ret (+ 2 p))))))
	;;
	ret
	))

(defclass json_prolog
  :super propertied-object
  :slots (query-srv next-srv finish-srv simple-srv finished attribute id))

(defmethod json_prolog
  (:init
   (&optional (name "json_prolog") &rest args)
   (setq id (send self :make-query-id)
		 finished t
		 query-srv  (format nil "/~a/query" name)
		 next-srv   (format nil "/~a/next_solution" name)
		 finish-srv (format nil "/~a/finish" name)
		 simple-srv (format nil "/~a/simple_query" name)))
  (:make-query-id
   ()
   (let ((tm (ros::time-now)))
	 (format nil "EUSLISP_QUERY_~a~0,9d" (send tm :sec)(send tm :nsec))))
  ;;
  (:query
   (query &key (mode :all) pose) ;; mode is :all or :incremental
   (when (not finished) (send self :finish))
   (let ((req (instance json_prolog_msgs::PrologQueryRequest :init)) res)
	 (cond ((eq mode :all)
			(send req :mode json_prolog_msgs::PrologQuery::*ALL_SOLUTIONS*))
		   ((eq mode :incremental)
			(send req :mode json_prolog_msgs::PrologQuery::*INCREMENTAL*))
		   (t
			(warn "mode should be :all or :incremental in :query")))
	 (when pose
	   (if (stringp pose) (setq pose (list pose)))
	   (setq query (send self :attribute-pose query pose)))
	 (send req :id (setq id (send self :make-query-id)))
	 (send req :query query)
	 (setq res (ros::service-call simple-srv req))
	 (ros::ros-info "prolog query> ~A" query)
	 (setq finished nil
		   attribute (if pose (append '(:pose) pose)))
	 (if (and (send res :ok) (eq mode :all))
		 (send self :solutions)
	   (send res :ok))
	 ))
  ;; for http://knowrob.org/kb/knowrob.owl\orientation
  (:attribute-pose
   (query objs &rest args)
   (dolist (obj objs)
	 (let (mtmp)
	   (setq query (format nil "~a,rdf_triple('http://knowrob.org/kb/knowrob.owl#orientation',~a,POSE_~a)" query obj obj))
	   (dotimes (i 4)
		 (dotimes (j 4)
		   (setq mtmp (format nil "~a_M~a~a" obj i j))
		   (setq query (format nil "~a,rdf_triple('http://knowrob.org/kb/knowrob.owl#m~a~a',POSE_~a,_~a), strip_literal_type(_~a, ~a)" query i j obj mtmp mtmp mtmp))))))
   query)
  (:extract-pose
   (answer objs)
   (let ((pose (make-array '(4 4) :element-type :float)) tmp)
	 (dolist (obj objs)
	   (dotimes (i 4)
		 (dotimes (j 4)
		   (setq tmp (assoc (format nil "~a_M~a~a" obj i j) answer :test #'string=))
		   (setf (aref pose i j) (read-from-string (cadr tmp)))
		   (if (eq (car answer) tmp)
			   (setq answer (cdr answer))
			 (delete tmp answer :count 1))
		   ))
	   (dotimes(i 3) (setf (aref pose i 3) (* 1000 (aref pose i 3)))) ;; m -> mm
	   (setq tmp (assoc (format nil "POSE_~a" obj) answer :test #'string=))
	   (setf (cadr tmp) (make-coords :4x4 pose :name (cadr tmp))))
	 answer))
  ;;
  ;;
  (:solutions
   ()
   (let (ans ret)
	 (while (setq ans (send self :next)) (push ans ret))
	 (nreverse ret)))
  (:next
   ()
   (if finished (return-from :next))
   (let ((req (instance json_prolog_msgs::PrologNextSolutionRequest :init))
		 res st answer)
	 (send req :id id)
	 (setq res (ros::service-call next-srv req))
	 (setq st (send res :status))
	 (cond
	  ((eq st json_prolog_msgs::PrologNextSolution::*NO_SOLUTION*)
	   (ros::ros-debug "No Solution") (send self :finish) nil)
	  ((eq st json_prolog_msgs::PrologNextSolution::*WRONG_ID*)
	   (ros::ros-info "Wrong Query ID") (send self :finish) nil)
	  ((eq st json_prolog_msgs::PrologNextSolution::*QUERY_FAILED*)
	   (ros::ros-info "Prolog query failed") (send self :finish) nil)
	  ((eq st json_prolog_msgs::PrologNextSolution::*OK*)
	   (setq answer (read-json (send res :solution)))
	   (case (car attribute)
		 (:pose (setq answer (send self :extract-pose answer (cdr attribute)))))
	   (ros::ros-info "prolog solution> ~A" answer)
	   answer ))
	 ))
  (:finish
   ()
   (if finished (return-from :finish))
   (let ((req (instance json_prolog_msgs::PrologFinishRequest :init)) res)
	 (send req :id id)
	 (setq res (ros::service-call finish-srv req))
	 (setq finished t attribute nil)
	 t ))
  )

;;
;; Utility function for making prolog query from list
;; ((a b (c d) e)(f g)) -> "a(b,c(d),e),f(g)"
;;
;; number  -> string
;; string  -> string
;; keyword -> string-upcase
;; symbol  -> string-downcase
;; vector  -> list []
;; list    -> map list->prolog
(defun list->prolog (lst)
  (flet ((conv-item (x)
    (cond ((numberp x) (string x))
          ((stringp x) x)
          ((keywordp x) (string-upcase x))
	  ((vectorp x) (list->prolog x))
          ((listp x) (list->prolog (list x)))
          (t (string-downcase x)))))
      (let (expr exprs ret)
        ;;
        (when (vectorp lst)
          (setq ret "[")
          (dotimes (i (length lst))
            (setq ret (format nil "~a~a" ret (conv-item (elt lst i))))
            (if (not (equal (1+ i) (length lst)))
              (setq ret (format nil "~a," ret))))
            (setq ret (format nil "~a]" ret)))
        ;;
        (when (listp lst)
          (dolist (l lst)
            (cond ((stringp l) (setq expr l))
              ((numberp l) (setq expr l))
              (t
               (setq expr (format nil "~a(~a" (car l) (conv-item (cadr l))))
               (dolist (item (cddr l))
                 (setq expr (format nil "~a,~a" expr (conv-item item))))
               (setq expr (format nil "~a)" expr))))
            (push expr exprs))
          (nreverse exprs)
          (setq ret (format nil "~a" (car exprs)))
          (dolist (ex (cdr exprs))
            (setq ret (format nil "~a, ~a" ret ex))))
        ret )))

;;
;; samples
;;

(defun init-json-prolog (&optional (namespace "json_prolog"))
  (unless (boundp '*prolog*)
	(setq *prolog* (instance json_prolog :init namespace))))

(defun prolog-sample1 ()
  (send *prolog* :query "member(A, [1, 2, 3, 4]), B = ['x', A]"))

(defun prolog-sample2 ()
  (send *prolog* :query "assert(friend(alice,bob))")
  (send *prolog* :query "assert(friend(alice,carol))")
  (print (send *prolog* :query "friend(alice,X)"))
  (send *prolog* :query "retract(friend(alice,bob))")
  (send *prolog* :query "retract(friend(alice,carol))"))

(defun prolog-sample3 () ;; sample for incremental mode
  (let (ans)
	(send *prolog* :query "member(A, [1, 2, 3, 4]), B is A * A."
		  :mode :incremental)
	(while (setq ans (send *prolog* :next))
	  (format nil "answer is ~a~%" ans)
	  (unix::sleep 1))
	))

;; with rosrun json_prolog json_prolog ias_semantic_map
(defun knowrob-sample1 ()
  (send *prolog* :query "rdfs_individual_of(Fridge, 'http://knowrob.org/kb/knowrob.owl#Refrigerator'), rdf_triple('http://knowrob.org/kb/knowrob.owl#orientation',Fridge,Pose), rdf_triple('http://knowrob.org/kb/knowrob.owl#m03',Pose,_M03), strip_literal_type(_M03, X), rdf_triple('http://knowrob.org/kb/knowrob.owl#m13',Pose,_M13), strip_literal_type(_M13, Y), rdf_triple('http://knowrob.org/kb/knowrob.owl#m23',Pose,_M23), strip_literal_type(_M23, Z)"))

(defun find-knowrob-objects (&optional (objtype "Refrigerator"))
  (let ((rdf-type (format nil "'http://knowrob.org/kb/knowrob.owl#~a'" objtype)))
	(send *prolog* :query
		  (list->prolog
		   (list `(rdfs_individual_of :obj ,rdf-type)))
		  :pose "OBJ")
	))

;; End of File
