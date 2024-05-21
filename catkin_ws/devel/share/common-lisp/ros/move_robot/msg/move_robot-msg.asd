
(cl:in-package :asdf)

(defsystem "move_robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "las_mes" :depends-on ("_package_las_mes"))
    (:file "_package_las_mes" :depends-on ("_package"))
    (:file "mission_plot_data" :depends-on ("_package_mission_plot_data"))
    (:file "_package_mission_plot_data" :depends-on ("_package"))
    (:file "ogm_data" :depends-on ("_package_ogm_data"))
    (:file "_package_ogm_data" :depends-on ("_package"))
    (:file "plot_data" :depends-on ("_package_plot_data"))
    (:file "_package_plot_data" :depends-on ("_package"))
  ))