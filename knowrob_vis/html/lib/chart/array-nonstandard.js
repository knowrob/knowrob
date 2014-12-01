// inspired by http://www.2ality.com/2013/12/array-prototype-find.html

    Array.prototype.find = function (predicate, thisValue) {
        var arr = Object(this);
        if (typeof predicate !== 'function') {
            throw new TypeError();
        }
        for(var i=0; i < arr.length; i++) {
            if (i in arr) {  // skip holes
                var elem = arr[i];
                if (predicate.call(thisValue, elem, i, arr)) {
                    return elem;  // (1)
                }
            }
        }
        return -1;  // (2)
    }

    Array.prototype.findIndex = function (predicate, thisValue) {
        var arr = Object(this);
        if (typeof predicate !== 'function') {
            throw new TypeError();
        }
        for(var i=0; i < arr.length; i++) {
            if (i in arr) {  // skip holes
                var elem = arr[i];
                if (predicate.call(thisValue, elem, i, arr)) {
                    return i;  // (1)
                }
            }
        }
        return -1;  // (2)
    }
