
(cl:in-package :asdf)

(defsystem "NavigationControl-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Navigate" :depends-on ("_package_Navigate"))
    (:file "_package_Navigate" :depends-on ("_package"))
  ))