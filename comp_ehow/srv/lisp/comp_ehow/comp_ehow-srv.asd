
(in-package :asdf)

(defsystem "comp_ehow-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "EhowToOWL" :depends-on ("_package"))
    (:file "_package_EhowToOWL" :depends-on ("_package"))
    ))
