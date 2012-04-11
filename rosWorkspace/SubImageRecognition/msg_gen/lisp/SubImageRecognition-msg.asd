
(cl:in-package :asdf)

(defsystem "SubImageRecognition-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ImgRecObject" :depends-on ("_package_ImgRecObject"))
    (:file "_package_ImgRecObject" :depends-on ("_package"))
  ))