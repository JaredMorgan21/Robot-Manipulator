classdef Color_Processor
    %COLOR_PROCESSOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        cam;
    end
    
    methods
        function self = Color_Processor(camera)
            self.cam = camera;
        end

        % Creates the HSV mask for the givin image, as well as hue,
        % saturation, and value minimums and maximums
        function img_mask = mask_HSV(self, img, hueMin, satMin, valMin, hueMax, satMax, valMax)
            hue = img(:, :, 1);
            saturation = img(:, :, 2);
            value = img(:, :, 3);
        
            hueMin = hue > hueMin/255;
            hueMax = hue < hueMax/255;
            satMin = saturation > satMin/255;
            satMax = saturation < satMax/255;
            valMin = value > valMin/255;
            valMax = value < valMax/255;

            hue(~(hueMin & hueMax)) = 0;
            saturation(~(satMin & satMax)) = 0;
            value(~(valMin & valMax)) = 0;
            
            mask = cat(3, hue, saturation, value);
            mask(:, 1:100, :) = 0;
            mask(:, end-100:end, :) = 0;
            mask(1:100, :, :) = 0;
            mask(end-100:end, :, :) = 0;
            img_mask = all(mask, 3);
        end
        
        % This function filters an image with a given mask.
        function filtered_img = filter_img(self, img, mask)
                d1 = img(:, :, 1);
                d2 = img(:, :, 2);
                d3 = img(:, :, 3);
                d1(~mask) = 0;
                d2(~mask) = 0;
                d3(~mask) = 0;
                
                filtered_img = cat(3, d1, d2, d3);
        end
        
        % This function searches for balls using canny edge detection.
        % Returns the image with the detected objects.
        function drawn_img = find_balls(self, img, mask, size)
                img_canny = edge(mask, 'Canny');
                img_canny = bwpropfilt(img_canny, 'Area', [size 5000]);
                boxes = regionprops(img_canny, 'BoundingBox');
                img_draw = img;
                
                for i = 1:height(boxes)
                    bb = boxes(i).BoundingBox;
                    img_draw = insertShape(img_draw, 'rectangle', [bb(1) bb(2) bb(3) bb(4)], 'Color', 'red', 'LineWidth', 2);
                end
                drawn_img = img_draw;
        end
        
        % This function draws centroids on the image, with the given mask
        % and size. This uses functions outlined in the lab document.
        function centroids = draw_centroids(self, img, mask, size)
            img_canny = edge(mask, 'Canny');
%             imshow(img_canny)
            img_canny = bwpropfilt(img_canny, 'Area', [size 5000]);
            balls = regionprops(img_canny, 'Centroid');
        
            img_draw = img;
        
            for i = 1:height(balls)
                c = balls(i).Centroid;
                img_draw = insertShape(img_draw, 'Circle', [c(1) c(2) 1], 'Color', 'green', 'LineWidth', 2);
            end
            centroids = img_draw;
        end
        
        % This function locates the centriods on the mask.
        function centroids = find_centroids(self, mask, size)
            imgCanny = edge(mask, 'Canny');
            imgCanny = bwpropfilt(imgCanny, 'Area', [size 5000]);
%             imshow(imgCanny)
            balls = regionprops(imgCanny, 'Centroid');

            cs = zeros(height(balls), 2);
            for i = 1:height(balls)
                cs(i, :) = balls(i).Centroid;
            end
            
            centroids = cs;
        end
        
        % This function detects the color of the ball based on the centrioid's hue, given
        % the HSV image and centrioid position.
        function position = ball_color(self, imgHSV, centroid)
            redHue = [0 10];
            redHighHue = [240 255];
            orangeHue = [10 30];
            yellowHue = [30 60];
            greenHue = [90 120];

            centroidPixel = imgHSV(round(centroid(2)), round(centroid(1)), 1) * 255
            if(centroidPixel >= redHue(1) && centroidPixel < redHue(2)) || (centroidPixel > redHighHue(1) && centroidPixel <= redHighHue(2))
                position = [0 125 10];
            elseif(centroidPixel > orangeHue(1) && centroidPixel < orangeHue(2))
                position = [0 75 10];
            elseif(centroidPixel > yellowHue(1) && centroidPixel < yellowHue(2))
                position = [0 -75 10];
            elseif(centroidPixel > greenHue(1) && centroidPixel < greenHue(2))
                position = [0 -125 10];
            else
                position = [50 -175 25];
%                 error("Ball color not known")
            end
        end
    end
end

