;; Auto-generated. Do not edit!


(when (boundp 'move_robot::las_mes)
  (if (not (find-package "MOVE_ROBOT"))
    (make-package "MOVE_ROBOT"))
  (shadow 'las_mes (find-package "MOVE_ROBOT")))
(unless (find-package "MOVE_ROBOT::LAS_MES")
  (make-package "MOVE_ROBOT::LAS_MES"))

(in-package "ROS")
;;//! \htmlinclude las_mes.msg.html


(defclass move_robot::las_mes
  :super ros::object
  :slots (_las_dist _las_dist_x _las_dist_y _las_angles ))

(defmethod move_robot::las_mes
  (:init
   (&key
    ((:las_dist __las_dist) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:las_dist_x __las_dist_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:las_dist_y __las_dist_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:las_angles __las_angles) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _las_dist __las_dist)
   (setq _las_dist_x __las_dist_x)
   (setq _las_dist_y __las_dist_y)
   (setq _las_angles __las_angles)
   self)
  (:las_dist
   (&optional __las_dist)
   (if __las_dist (setq _las_dist __las_dist)) _las_dist)
  (:las_dist_x
   (&optional __las_dist_x)
   (if __las_dist_x (setq _las_dist_x __las_dist_x)) _las_dist_x)
  (:las_dist_y
   (&optional __las_dist_y)
   (if __las_dist_y (setq _las_dist_y __las_dist_y)) _las_dist_y)
  (:las_angles
   (&optional __las_angles)
   (if __las_angles (setq _las_angles __las_angles)) _las_angles)
  (:serialization-length
   ()
   (+
    ;; float64[] _las_dist
    (* 8    (length _las_dist)) 4
    ;; float64[] _las_dist_x
    (* 8    (length _las_dist_x)) 4
    ;; float64[] _las_dist_y
    (* 8    (length _las_dist_y)) 4
    ;; float64[] _las_angles
    (* 8    (length _las_angles)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _las_dist
     (write-long (length _las_dist) s)
     (dotimes (i (length _las_dist))
       (sys::poke (elt _las_dist i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _las_dist_x
     (write-long (length _las_dist_x) s)
     (dotimes (i (length _las_dist_x))
       (sys::poke (elt _las_dist_x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _las_dist_y
     (write-long (length _las_dist_y) s)
     (dotimes (i (length _las_dist_y))
       (sys::poke (elt _las_dist_y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _las_angles
     (write-long (length _las_angles) s)
     (dotimes (i (length _las_angles))
       (sys::poke (elt _las_angles i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _las_dist
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _las_dist (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _las_dist i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _las_dist_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _las_dist_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _las_dist_x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _las_dist_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _las_dist_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _las_dist_y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _las_angles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _las_angles (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _las_angles i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get move_robot::las_mes :md5sum-) "aad5a2de89d4ae32cf5dfd4ccf601549")
(setf (get move_robot::las_mes :datatype-) "move_robot/las_mes")
(setf (get move_robot::las_mes :definition-)
      "float64[] las_dist
float64[] las_dist_x
float64[] las_dist_y
float64[] las_angles
# float64[] velocities

")



(provide :move_robot/las_mes "aad5a2de89d4ae32cf5dfd4ccf601549")


