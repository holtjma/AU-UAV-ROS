
(cl:in-package :asdf)

(defsystem "AU_UAV_ROS-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CreateSimulatedPlane" :depends-on ("_package_CreateSimulatedPlane"))
    (:file "_package_CreateSimulatedPlane" :depends-on ("_package"))
    (:file "RequestPlaneID" :depends-on ("_package_RequestPlaneID"))
    (:file "_package_RequestPlaneID" :depends-on ("_package"))
    (:file "GoToWaypoint" :depends-on ("_package_GoToWaypoint"))
    (:file "_package_GoToWaypoint" :depends-on ("_package"))
  ))