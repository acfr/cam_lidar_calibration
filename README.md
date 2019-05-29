This is a template for a nodelet. There are two parameters that can be changed throughout the code and configuration files to customise this for a new class

nodelet_class is the name of the package, and the name of the class used as the nodelet
nodelet_namespace is the name of the c++ namespace used for the project

To change these throughout the code, the following command line arguments can be used. 
You need to change the uppercase NODELET_CLASS to what you want the class and nodelet package to be called, and change NODELET_NAMESPACE to the name space you want to use.

find . -exec sed -i 's/nodelet_class/NODELET_CLASS/g' {} \;

find . -exec sed -i 's/nodelet_namespace/NODELET_NAMESPACE/g' {} \;