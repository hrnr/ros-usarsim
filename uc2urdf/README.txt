------------------------------------------------------------------------------------------
To compile: make
------------------------------------------------------------------------------------------
To run: To run the program some arguments need to be passed to the executable.

./uc2urdf -input oldurdf.xml -output newurdf.xml

oldurdf.xml: The "name" of the original urdf file built from tf data.
The program will automatically search for this file in ../usarsim_inf/urdf

newurdf.xml: The "name" of the output urdf file.
The program will automatically save the new file in ../etc/
------------------------------------------------------------------------------------------
Documentation: To generate the documentation (html and latex) for the code, 
place yourself in the 'doc' directory and run 'doxygen Doxyfile'.
------------------------------------------------------------------------------------------

