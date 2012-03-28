
(cl:in-package :asdf)

(defsystem "subSim-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MotorMessage" :depends-on ("_package_MotorMessage"))
    (:file "_package_MotorMessage" :depends-on ("_package"))
    (:file "mixer" :depends-on ("_package_mixer"))
    (:file "_package_mixer" :depends-on ("_package"))
  ))