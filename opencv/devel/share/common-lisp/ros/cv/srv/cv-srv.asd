
(cl:in-package :asdf)

(defsystem "cv-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "date" :depends-on ("_package_date"))
    (:file "_package_date" :depends-on ("_package"))
  ))