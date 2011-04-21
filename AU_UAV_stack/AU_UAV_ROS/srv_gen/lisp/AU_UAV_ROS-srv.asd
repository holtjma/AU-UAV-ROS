
(cl:in-package :asdf)

(defsystem "AU_UAV_ROS-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CreateSimulatedPlane" :depends-on ("_package_CreateSimulatedPlane"))
    (:file "_package_CreateSimulatedPlane" :depends-on ("_package"))
    (:file "RequestPlaneID" :depends-on ("_package_RequestPlaneID"))
    (:file "_package_RequestPlaneID" :depends-on ("_package"))
    (:file "AvoidCollision" :depends-on ("_package_AvoidCollision"))
    (:file "_package_AvoidCollision" :depends-on ("_package"))
  ))