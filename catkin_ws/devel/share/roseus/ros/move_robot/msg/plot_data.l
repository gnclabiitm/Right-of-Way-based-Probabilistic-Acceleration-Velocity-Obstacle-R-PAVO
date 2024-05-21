;; Auto-generated. Do not edit!


(when (boundp 'move_robot::plot_data)
  (if (not (find-package "MOVE_ROBOT"))
    (make-package "MOVE_ROBOT"))
  (shadow 'plot_data (find-package "MOVE_ROBOT")))
(unless (find-package "MOVE_ROBOT::PLOT_DATA")
  (make-package "MOVE_ROBOT::PLOT_DATA"))

(in-package "ROS")
;;//! \htmlinclude plot_data.msg.html


(defclass move_robot::plot_data
  :super ros::object
  :slots (_curr_velocity_x _curr_velocity_y _des_velocity_x _des_velocity_y _t_iter _delta _t_horizon _avoid_i _a_long _a_lat _vel_mag _a_long_lv _a_lat_wv _accel ))

(defmethod move_robot::plot_data
  (:init
   (&key
    ((:curr_velocity_x __curr_velocity_x) 0.0)
    ((:curr_velocity_y __curr_velocity_y) 0.0)
    ((:des_velocity_x __des_velocity_x) 0.0)
    ((:des_velocity_y __des_velocity_y) 0.0)
    ((:t_iter __t_iter) 0.0)
    ((:delta __delta) 0.0)
    ((:t_horizon __t_horizon) 0.0)
    ((:avoid_i __avoid_i) 0.0)
    ((:a_long __a_long) 0.0)
    ((:a_lat __a_lat) 0.0)
    ((:vel_mag __vel_mag) 0.0)
    ((:a_long_lv __a_long_lv) 0.0)
    ((:a_lat_wv __a_lat_wv) 0.0)
    ((:accel __accel) 0.0)
    )
   (send-super :init)
   (setq _curr_velocity_x (float __curr_velocity_x))
   (setq _curr_velocity_y (float __curr_velocity_y))
   (setq _des_velocity_x (float __des_velocity_x))
   (setq _des_velocity_y (float __des_velocity_y))
   (setq _t_iter (float __t_iter))
   (setq _delta (float __delta))
   (setq _t_horizon (float __t_horizon))
   (setq _avoid_i (float __avoid_i))
   (setq _a_long (float __a_long))
   (setq _a_lat (float __a_lat))
   (setq _vel_mag (float __vel_mag))
   (setq _a_long_lv (float __a_long_lv))
   (setq _a_lat_wv (float __a_lat_wv))
   (setq _accel (float __accel))
   self)
  (:curr_velocity_x
   (&optional __curr_velocity_x)
   (if __curr_velocity_x (setq _curr_velocity_x __curr_velocity_x)) _curr_velocity_x)
  (:curr_velocity_y
   (&optional __curr_velocity_y)
   (if __curr_velocity_y (setq _curr_velocity_y __curr_velocity_y)) _curr_velocity_y)
  (:des_velocity_x
   (&optional __des_velocity_x)
   (if __des_velocity_x (setq _des_velocity_x __des_velocity_x)) _des_velocity_x)
  (:des_velocity_y
   (&optional __des_velocity_y)
   (if __des_velocity_y (setq _des_velocity_y __des_velocity_y)) _des_velocity_y)
  (:t_iter
   (&optional __t_iter)
   (if __t_iter (setq _t_iter __t_iter)) _t_iter)
  (:delta
   (&optional __delta)
   (if __delta (setq _delta __delta)) _delta)
  (:t_horizon
   (&optional __t_horizon)
   (if __t_horizon (setq _t_horizon __t_horizon)) _t_horizon)
  (:avoid_i
   (&optional __avoid_i)
   (if __avoid_i (setq _avoid_i __avoid_i)) _avoid_i)
  (:a_long
   (&optional __a_long)
   (if __a_long (setq _a_long __a_long)) _a_long)
  (:a_lat
   (&optional __a_lat)
   (if __a_lat (setq _a_lat __a_lat)) _a_lat)
  (:vel_mag
   (&optional __vel_mag)
   (if __vel_mag (setq _vel_mag __vel_mag)) _vel_mag)
  (:a_long_lv
   (&optional __a_long_lv)
   (if __a_long_lv (setq _a_long_lv __a_long_lv)) _a_long_lv)
  (:a_lat_wv
   (&optional __a_lat_wv)
   (if __a_lat_wv (setq _a_lat_wv __a_lat_wv)) _a_lat_wv)
  (:accel
   (&optional __accel)
   (if __accel (setq _accel __accel)) _accel)
  (:serialization-length
   ()
   (+
    ;; float64 _curr_velocity_x
    8
    ;; float64 _curr_velocity_y
    8
    ;; float64 _des_velocity_x
    8
    ;; float64 _des_velocity_y
    8
    ;; float64 _t_iter
    8
    ;; float64 _delta
    8
    ;; float64 _t_horizon
    8
    ;; float64 _avoid_i
    8
    ;; float64 _a_long
    8
    ;; float64 _a_lat
    8
    ;; float64 _vel_mag
    8
    ;; float64 _a_long_lv
    8
    ;; float64 _a_lat_wv
    8
    ;; float64 _accel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _curr_velocity_x
       (sys::poke _curr_velocity_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _curr_velocity_y
       (sys::poke _curr_velocity_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _des_velocity_x
       (sys::poke _des_velocity_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _des_velocity_y
       (sys::poke _des_velocity_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t_iter
       (sys::poke _t_iter (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta
       (sys::poke _delta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t_horizon
       (sys::poke _t_horizon (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _avoid_i
       (sys::poke _avoid_i (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a_long
       (sys::poke _a_long (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a_lat
       (sys::poke _a_lat (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vel_mag
       (sys::poke _vel_mag (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a_long_lv
       (sys::poke _a_long_lv (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a_lat_wv
       (sys::poke _a_lat_wv (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _accel
       (sys::poke _accel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _curr_velocity_x
     (setq _curr_velocity_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _curr_velocity_y
     (setq _curr_velocity_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _des_velocity_x
     (setq _des_velocity_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _des_velocity_y
     (setq _des_velocity_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t_iter
     (setq _t_iter (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta
     (setq _delta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t_horizon
     (setq _t_horizon (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _avoid_i
     (setq _avoid_i (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a_long
     (setq _a_long (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a_lat
     (setq _a_lat (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vel_mag
     (setq _vel_mag (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a_long_lv
     (setq _a_long_lv (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a_lat_wv
     (setq _a_lat_wv (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _accel
     (setq _accel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get move_robot::plot_data :md5sum-) "6b90cbb3473e15ae3fe0a7a90e7e54aa")
(setf (get move_robot::plot_data :datatype-) "move_robot/plot_data")
(setf (get move_robot::plot_data :definition-)
      "float64 curr_velocity_x
float64 curr_velocity_y
float64 des_velocity_x
float64 des_velocity_y
float64 t_iter
float64 delta
float64 t_horizon
float64 avoid_i
float64 a_long
float64 a_lat
float64 vel_mag
float64 a_long_lv
float64 a_lat_wv
float64 accel
")



(provide :move_robot/plot_data "6b90cbb3473e15ae3fe0a7a90e7e54aa")


