
(cl:in-package :asdf)

(defsystem "move_robot-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "OGM" :depends-on ("_package_OGM"))
    (:file "_package_OGM" :depends-on ("_package"))
    (:file "SyncMotion" :depends-on ("_package_SyncMotion"))
    (:file "_package_SyncMotion" :depends-on ("_package"))
    (:file "xycoord" :depends-on ("_package_xycoord"))
    (:file "_package_xycoord" :depends-on ("_package"))
  ))