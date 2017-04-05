 clear;
 close all;
 I = imread('moon.tif');
 %I = rgb2gray(I);
 I = im2double(I);
 OriginalImage = I;
 [k,l] = size(I);
 I = imresize(I, [512 512]);
 n = 8
 T = dctmtx(n);
 dct = @(block_struct) T * block_struct.data * T';
 B = blockproc(I,[n n],dct);
 B2 = B;
 c = 10;
 [M,N]=size(B);
  for x = 1:n
    for y = 1:n
                   
        if x < (n+1)/2 && y < (n+1)/2
             mask(x,y) = 1; 
        else
             mask(x,y) = 0; 
        end
    end
 end         

 selection_matrix = repmat(mask,512/n,512/n);
 B2 = B2.*selection_matrix;
 %B2= blockproc(B,[n n],@(block_struct) mask .* block_struct.data);
 
% B2 = B2./max(max(max(B2)));
%  for x = 1:M
%      for y = 1:N
%          m=double(B2(x,y));
%          B2(x,y)=c*log10(1+m);
%      end
%   end

 B2 = imresize(B2,[512 512]);
 
% JPEG Standard Quantization Array
Quantz=[16 11 10 16  24  40  51  61
         12 12 14 19  26  58  60  55
         14 13 16 24  40  57  69  56
         14 17 22 29  51  87  80  62
         18 22 37 56  68 109 103  77
         24 35 55 64  81 104 113  92
         49 64 78 87 103 121 120 101
         72 92 95 98 112 100 103 99 ];
     
% Quantz= [8 12 14 16 20 26.5 37 64
%          12 18 21 26 31 40 57 98
%          14 22 25 29 36  47 66 113
%          16 26  29 35 43 55  78 134
%          21 31 36 43 53 68 96 164
%          27 40 47 55  68 88 124 212
%          37 57 66 78 96 124 176 300
%          64 98 113 134 164 212 300 512 ];

prev_val = -1;
count = 0;
output_size = 0;


ResizedQuantizer = repmat(uint8(ceil(double(Quantz)./90000)), 512/8 , 512/8);
 
QuantizedImage = round(B2./double(ResizedQuantizer));

 for x = 1:512
    for y = 1:512
       if prev_val ~= QuantizedImage(x,y)
           if count ~= 0
               output_size = output_size + 2;
           end
           count = 1;
       else
           count = count +1;
       end
       prev_val = QuantizedImage(x,y);
    end
 end     
output_size = output_size +2
compression_ratio = (512*512)/output_size
DeQuantizedImage = QuantizedImage.*double(ResizedQuantizer);
 
invdct = @(block_struct) T' * block_struct.data * T;
I2 = blockproc(DeQuantizedImage,[n n],invdct);
ReconstructedImage = imresize(I2,[k l]);
error = OriginalImage - ReconstructedImage;
MSE = sum(sum(error .* error))/(k*l);
PSNR = 20*log10(max(max(max(OriginalImage)))) - 10*log10(MSE);

textstring = ['N : ' num2str(n) ' X ' num2str(n) ' , Compression Ratio : ' num2str(compression_ratio) ': 1 , PSNR :' num2str(PSNR) ];
position =  [1 50];
figure;
imshow(ReconstructedImage);
title(textstring);
figure;
imshow(OriginalImage);

