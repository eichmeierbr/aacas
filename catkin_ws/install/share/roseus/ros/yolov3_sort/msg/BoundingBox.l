;; Auto-generated. Do not edit!


(when (boundp 'yolov3_sort::BoundingBox)
  (if (not (find-package "YOLOV3_SORT"))
    (make-package "YOLOV3_SORT"))
  (shadow 'BoundingBox (find-package "YOLOV3_SORT")))
(unless (find-package "YOLOV3_SORT::BOUNDINGBOX")
  (make-package "YOLOV3_SORT::BOUNDINGBOX"))

(in-package "ROS")
;;//! \htmlinclude BoundingBox.msg.html


(defclass yolov3_sort::BoundingBox
  :super ros::object
  :slots (_label _idx _xmin _ymin _xmax _ymax ))

(defmethod yolov3_sort::BoundingBox
  (:init
   (&key
    ((:label __label) 0)
    ((:idx __idx) 0)
    ((:xmin __xmin) 0.0)
    ((:ymin __ymin) 0.0)
    ((:xmax __xmax) 0.0)
    ((:ymax __ymax) 0.0)
    )
   (send-super :init)
   (setq _label (round __label))
   (setq _idx (round __idx))
   (setq _xmin (float __xmin))
   (setq _ymin (float __ymin))
   (setq _xmax (float __xmax))
   (setq _ymax (float __ymax))
   self)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:idx
   (&optional __idx)
   (if __idx (setq _idx __idx)) _idx)
  (:xmin
   (&optional __xmin)
   (if __xmin (setq _xmin __xmin)) _xmin)
  (:ymin
   (&optional __ymin)
   (if __ymin (setq _ymin __ymin)) _ymin)
  (:xmax
   (&optional __xmax)
   (if __xmax (setq _xmax __xmax)) _xmax)
  (:ymax
   (&optional __ymax)
   (if __ymax (setq _ymax __ymax)) _ymax)
  (:serialization-length
   ()
   (+
    ;; int64 _label
    8
    ;; int64 _idx
    8
    ;; float64 _xmin
    8
    ;; float64 _ymin
    8
    ;; float64 _xmax
    8
    ;; float64 _ymax
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _label
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _label (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _label) (= (length (_label . bv)) 2)) ;; bignum
              (write-long (ash (elt (_label . bv) 0) 0) s)
              (write-long (ash (elt (_label . bv) 1) -1) s))
             ((and (class _label) (= (length (_label . bv)) 1)) ;; big1
              (write-long (elt (_label . bv) 0) s)
              (write-long (if (>= _label 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _label s)(write-long (if (>= _label 0) 0 #xffffffff) s)))
     ;; int64 _idx
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _idx (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _idx) (= (length (_idx . bv)) 2)) ;; bignum
              (write-long (ash (elt (_idx . bv) 0) 0) s)
              (write-long (ash (elt (_idx . bv) 1) -1) s))
             ((and (class _idx) (= (length (_idx . bv)) 1)) ;; big1
              (write-long (elt (_idx . bv) 0) s)
              (write-long (if (>= _idx 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _idx s)(write-long (if (>= _idx 0) 0 #xffffffff) s)))
     ;; float64 _xmin
       (sys::poke _xmin (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ymin
       (sys::poke _ymin (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _xmax
       (sys::poke _xmax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ymax
       (sys::poke _ymax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _label
#+(or :alpha :irix6 :x86_64)
      (setf _label (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _label (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _idx
#+(or :alpha :irix6 :x86_64)
      (setf _idx (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _idx (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _xmin
     (setq _xmin (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ymin
     (setq _ymin (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _xmax
     (setq _xmax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ymax
     (setq _ymax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get yolov3_sort::BoundingBox :md5sum-) "167647b523a0c139be92f837afe9e9e7")
(setf (get yolov3_sort::BoundingBox :datatype-) "yolov3_sort/BoundingBox")
(setf (get yolov3_sort::BoundingBox :definition-)
      "int64 label
int64 idx
float64 xmin
float64 ymin
float64 xmax
float64 ymax

")



(provide :yolov3_sort/BoundingBox "167647b523a0c139be92f837afe9e9e7")


