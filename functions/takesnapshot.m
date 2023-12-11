function image = takesnapshot(sub_image,ID)
    %TAKESNAPSHOT 
    
    imagemsg = receive(sub_image);
    image = readImage(imagemsg);
    imagename = strcat('data/images/image_',num2str(ID),'.png');
    imwrite(image,imagename)
end

