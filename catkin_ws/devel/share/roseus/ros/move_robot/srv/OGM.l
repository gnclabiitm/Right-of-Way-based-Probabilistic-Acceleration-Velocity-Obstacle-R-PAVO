;; Auto-generated. Do not edit!


(when (boundp 'move_robot::OGM)
  (if (not (find-package "MOVE_ROBOT"))
    (make-package "MOVE_ROBOT"))
  (shadow 'OGM (find-package "MOVE_ROBOT")))
(unless (find-package "MOVE_ROBOT::OGM")
  (make-package "MOVE_ROBOT::OGM"))
(unless (find-package "MOVE_ROBOT::OGMREQUEST")
  (make-package "MOVE_ROBOT::OGMREQUEST"))
(unless (find-package "MOVE_ROBOT::OGMRESPONSE")
  (make-package "MOVE_ROBOT::OGMRESPONSE"))

(in-package "ROS")





(defclass move_robot::OGMRequest
  :super ros::object
  :slots (_tb3_id _ogm_node_num ))

(defmethod move_robot::OGMRequest
  (:init
   (&key
    ((:tb3_id __tb3_id) "")
    ((:ogm_node_num __ogm_node_num) "")
    )
   (send-super :init)
   (setq _tb3_id (string __tb3_id))
   (setq _ogm_node_num (string __ogm_node_num))
   self)
  (:tb3_id
   (&optional __tb3_id)
   (if __tb3_id (setq _tb3_id __tb3_id)) _tb3_id)
  (:ogm_node_num
   (&optional __ogm_node_num)
   (if __ogm_node_num (setq _ogm_node_num __ogm_node_num)) _ogm_node_num)
  (:serialization-length
   ()
   (+
    ;; string _tb3_id
    4 (length _tb3_id)
    ;; string _ogm_node_num
    4 (length _ogm_node_num)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _tb3_id
       (write-long (length _tb3_id) s) (princ _tb3_id s)
     ;; string _ogm_node_num
       (write-long (length _ogm_node_num) s) (princ _ogm_node_num s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _tb3_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _tb3_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _ogm_node_num
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ogm_node_num (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass move_robot::OGMResponse
  :super ros::object
  :slots (_dummy ))

(defmethod move_robot::OGMResponse
  (:init
   (&key
    ((:dummy __dummy) "")
    )
   (send-super :init)
   (setq _dummy (string __dummy))
   self)
  (:dummy
   (&optional __dummy)
   (if __dummy (setq _dummy __dummy)) _dummy)
  (:serialization-length
   ()
   (+
    ;; string _dummy
    4 (length _dummy)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _dummy
       (write-long (length _dummy) s) (princ _dummy s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _dummy
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _dummy (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass move_robot::OGM
  :super ros::object
  :slots ())

(setf (get move_robot::OGM :md5sum-) "a3ef7e635198434f8c5d1fcc111d7ba4")
(setf (get move_robot::OGM :datatype-) "move_robot/OGM")
(setf (get move_robot::OGM :request) move_robot::OGMRequest)
(setf (get move_robot::OGM :response) move_robot::OGMResponse)

(defmethod move_robot::OGMRequest
  (:response () (instance move_robot::OGMResponse :init)))

(setf (get move_robot::OGMRequest :md5sum-) "a3ef7e635198434f8c5d1fcc111d7ba4")
(setf (get move_robot::OGMRequest :datatype-) "move_robot/OGMRequest")
(setf (get move_robot::OGMRequest :definition-)
      "string tb3_id
string ogm_node_num
---
string dummy
")

(setf (get move_robot::OGMResponse :md5sum-) "a3ef7e635198434f8c5d1fcc111d7ba4")
(setf (get move_robot::OGMResponse :datatype-) "move_robot/OGMResponse")
(setf (get move_robot::OGMResponse :definition-)
      "string tb3_id
string ogm_node_num
---
string dummy
")



(provide :move_robot/OGM "a3ef7e635198434f8c5d1fcc111d7ba4")


