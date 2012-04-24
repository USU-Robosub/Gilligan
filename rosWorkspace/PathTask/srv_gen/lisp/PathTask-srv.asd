
(cl:in-package :asdf)

(defsystem "PathTask-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Toggle" :depends-on ("_package_Toggle"))
    (:file "_package_Toggle" :depends-on ("_package"))
  ))