"I pledge on my honor that I have not received any unauthorized assistance on this project and that I have properly acknowledged all sources of material that was not solely produced by me."

Electronically signed by Lakshman Kumar Kuttuva Nandhakumar
Date: 5/14/2017

API's used
OpenCV : Contour Detection, ORB Feature Detection/Matching, I/O operations and Visualization .

Rest of the techniques were completely implemented by me in C++ as a part of the ENEE631 course.

To compile the code, follow the instructions below

- mkdir build
- cd build
- cmake ..
- make

For Document Extraction
Upload the captured image to the images/document_scan folder and name the image as scanned_image.jpg

For image registration

Upload the images of the reference document and filled document to the images/image_registration folder and name them as reference_image.jpg and scanned_image.jpg respectively

To run the code, go to the build directory and do
" ./mobile_scanner 0 " for running just document extraction
" ./mobile_scanner 1 " for running both document extraction and image registration to extract text in filled forms and replacing it in the reference form


