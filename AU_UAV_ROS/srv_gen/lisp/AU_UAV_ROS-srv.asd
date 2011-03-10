
(cl:in-package :asdf)

(defsystem "AU_UAV_ROS-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AvoidCollision" :depends-on ("_package_AvoidCollision"))
    (:file "_package_AvoidCollision" :depends-on ("_package"))
  ))