
(cl:in-package :asdf)

(defsystem "AU_UAV_ROS-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "TelemetryUpdate" :depends-on ("_package_TelemetryUpdate"))
    (:file "_package_TelemetryUpdate" :depends-on ("_package"))
    (:file "Command" :depends-on ("_package_Command"))
    (:file "_package_Command" :depends-on ("_package"))
  ))