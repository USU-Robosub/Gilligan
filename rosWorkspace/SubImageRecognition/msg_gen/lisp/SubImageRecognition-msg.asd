
(cl:in-package :asdf)

(defsystem "SubImageRecognition-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "OrangeRectangle" :depends-on ("_package_OrangeRectangle"))
    (:file "_package_OrangeRectangle" :depends-on ("_package"))
  ))