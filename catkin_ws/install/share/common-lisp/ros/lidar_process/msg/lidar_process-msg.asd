
(cl:in-package :asdf)

(defsystem "lidar_process-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "tracked_obj" :depends-on ("_package_tracked_obj"))
    (:file "_package_tracked_obj" :depends-on ("_package"))
    (:file "tracked_obj_arr" :depends-on ("_package_tracked_obj_arr"))
    (:file "_package_tracked_obj_arr" :depends-on ("_package"))
  ))