;; Auto-generated. Do not edit!


(when (boundp 'move_robot::mission_plot_data)
  (if (not (find-package "MOVE_ROBOT"))
    (make-package "MOVE_ROBOT"))
  (shadow 'mission_plot_data (find-package "MOVE_ROBOT")))
(unless (find-package "MOVE_ROBOT::MISSION_PLOT_DATA")
  (make-package "MOVE_ROBOT::MISSION_PLOT_DATA"))

(in-package "ROS")
;;//! \htmlinclude mission_plot_data.msg.html


(defclass move_robot::mission_plot_data
  :super ros::object
  :slots (_t_mission _d_min _t_comp _a_rms ))

(defmethod move_robot::mission_plot_data
  (:init
   (&key
    ((:t_mission __t_mission) 0.0)
    ((:d_min __d_min) 0.0)
    ((:t_comp __t_comp) 0.0)
    ((:a_rms __a_rms) 0.0)
    )
   (send-super :init)
   (setq _t_mission (float __t_mission))
   (setq _d_min (float __d_min))
   (setq _t_comp (float __t_comp))
   (setq _a_rms (float __a_rms))
   self)
  (:t_mission
   (&optional __t_mission)
   (if __t_mission (setq _t_mission __t_mission)) _t_mission)
  (:d_min
   (&optional __d_min)
   (if __d_min (setq _d_min __d_min)) _d_min)
  (:t_comp
   (&optional __t_comp)
   (if __t_comp (setq _t_comp __t_comp)) _t_comp)
  (:a_rms
   (&optional __a_rms)
   (if __a_rms (setq _a_rms __a_rms)) _a_rms)
  (:serialization-length
   ()
   (+
    ;; float64 _t_mission
    8
    ;; float64 _d_min
    8
    ;; float64 _t_comp
    8
    ;; float64 _a_rms
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _t_mission
       (sys::poke _t_mission (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d_min
       (sys::poke _d_min (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t_comp
       (sys::poke _t_comp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a_rms
       (sys::poke _a_rms (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _t_mission
     (setq _t_mission (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d_min
     (setq _d_min (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t_comp
     (setq _t_comp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a_rms
     (setq _a_rms (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get move_robot::mission_plot_data :md5sum-) "cc7d6c5759137cf046d2c010b9c936e4")
(setf (get move_robot::mission_plot_data :datatype-) "move_robot/mission_plot_data")
(setf (get move_robot::mission_plot_data :definition-)
      "float64 t_mission
float64 d_min
float64 t_comp
float64 a_rms
")



(provide :move_robot/mission_plot_data "cc7d6c5759137cf046d2c010b9c936e4")


