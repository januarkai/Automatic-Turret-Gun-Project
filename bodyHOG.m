clc
clear all
%inisialisasi parameter
% rosinit('10.252.128.115',11311);
% node = robotics.ros.Node('/test_node_1');
bodyDetector = vision.PeopleDetector('ClassificationModel','UprightPeople_128x64','MergeDetections',true);
bodyDetector.MinSize = [128 64];
%bodyDetector.MergeDetections;
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
%cam = webcam('Logitech HD Webcam C615')
cam = webcam;
videoFrame = snapshot(cam);
%image(videoFrame);
%figure;imshow(videoFrame,[270,325]);
frameSize = size(videoFrame)
%htextinsCent = vision.TextInserter('Text', '+      X:%4d, Y:%4d', ... % set text for centroid
 %                                   'LocationSource', 'Input port', ...
  %                                  'Color', [1 1 0], ... // yellow color
   %                                 'Font', 'Courier New', ...
    %                                'FontSize', 14);
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
runLoop = true;
numPts = 0;
frameCount = 0;
Lastelapsed=0;
% s=serial('COM4','BaudRate',9600);
% fopen(s);
start=tic;
while runLoop && frameCount < inf

    % Get the next frame.
    videoFrame = snapshot(cam);
    
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;

    if numPts < 10
        % Detection mode.
        bbox = bodyDetector.step(videoFrameGray);
        %centers = round(bbox(:, 1:2) + bbox(:, 3:4) / 2);
%         pub = rospublisher('/nilai','geometry_msgs/Point');
%         pause(1/100);
%         msg = rosmessage(pub);
        if ~isempty(bbox)
            % Find corner points inside the detected region.
            %centers = round(bbox(:, 1:2) + bbox(:, 3:4) / 2);
            points = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox(1, :));

            % Re-initialize the point tracker.
            xyPoints = points.Location;
            numPts = size(xyPoints,1);
            release(pointTracker);
            initialize(pointTracker, xyPoints, videoFrameGray);

            % Save a copy of the points.
            

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            %bboxPoints = bbox2points(bbox(1, :));
            oldPoints = xyPoints;
            bboxPoints = bbox2points(bbox(1, :));
            bboxPolygon = reshape(bboxPoints', 1, []);
            %bboxPoints = insertMarker (RGB,'centroid');
            

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            
            %centers = round(bboxPolygon(:, 1:2)+210)%+ bboxPolygon(:, 3:4)/2)
            centers = round((bboxPolygon(:, 1:2)/2)+( bboxPolygon(:, 3:4)/2));
            %x=sprintf('%u',centers(1))
            %y=sprintf('%u',centers(2))
%             pix=mat2str(centers);
%             pause(2);
%             fprintf(s,pix);
            xA=centers(1);
            yA=centers(2);
            x=round((320-xA)/9.78)
            yA1=(yA-240)/240;
            y=round(atan(yA1/5))
%             msg.X=x;
%             msg.Y=y;
            %a=int2str(x);
            %b=int2str(y);
            %center = insertMarker(bboxPolygon,'centroid');

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);
            videoFrame = insertMarker(videoFrame,centers,'x','Color', 'white','size',10 );
            videoFrame = insertText(videoFrame,centers,'tembak sini','BoxColor','green','TextColor','red');
            %videoFrame = vision.TextInserter('Text', '+      X:%4d, Y:%4d', ... % set text for centroid
             %                       centers, 'Input port', ...
              %                      'Color', [1 1 0], ... // yellow color
               %                    'FontSize', 14);
            % Display detected corners.
            stringvar=['D:\Progress PA2\framedetection\frame',num2str(frameCount)];
            stringvar=join(stringvar);
            stingvar=[stringvar,'.jpg'];
            imwrite(videoFrame,join(stingvar));
            videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
        end

    else
        % Tracking mode.
        [xyPoints, isFound] = step(pointTracker, videoFrameGray);
        visiblePoints = xyPoints(isFound, :);
        oldInliers = oldPoints(isFound, :);

        numPts = size(visiblePoints, 1);

        if numPts >= 10
             [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
              oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);
            %centers = round(bbox(:, 1:2) + bbox(:, 3:4) / 2);

            % Apply the transformation to the bounding box.
            bboxPoints = transformPointsForward(xform, bboxPoints);
            

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);
            %centers = round(bboxPolygon(:, 1:2)+210 )%+ bboxPolygon(:, 3:4)/2)
            centers = round((bboxPolygon(:, 1:2)/2)+( bboxPolygon(:, 3:4)/2));
            %x=sprintf('%u',centers(1))
            %y=sprintf('%u',centers(2))
%             pix=mat2str(centers);
%             pause(2);
%             fprintf(s,pix);
            xA=centers(1);
            yA=centers(2);
            x=round((320-xA)/9.78)
            yA1=(yA-240)/240;
            y=round(atan(yA1/5))
%             msg.X=x;
%             msg.Y=y;
            %a=int2str(x);
            %b=int2str(y);
            

            % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);
            videoFrame = insertMarker(videoFrame,centers,'x','Color', 'white','size',10 );
            videoFrame = insertText(videoFrame,centers,'tembak sini','BoxColor','green','TextColor','red');
            %videoFrame = vision.TextInserter('Text', '+      X:%4d, Y:%4d', ... % set text for centroid
            %                        centers, 'Input port', ...
             %                       'Color', [1 1 0], ... // yellow color
              %                     'FontSize', 14);
            % Display tracked points.
            stringvar=['D:\Progress PA2\framedetection\frame',num2str(frameCount)];
            stringvar=join(stringvar);
            stingvar=[stringvar,'.jpg'];
            imwrite(videoFrame,join(stingvar));
            videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');

            % Reset the points.
            oldPoints = visiblePoints;
            setPoints(pointTracker, oldPoints);
        end

    end
    %pix2=fscanf(s)
%     send(pub,msg)
%     rostopic list
%     rosnode list
    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);
    elapsed=toc(start);
    exctime=elapsed-Lastelapsed;
    Lastelapsed=elapsed;
    disp(['it has been ',num2str(exctime),' seconds'])
    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
end
%fclose(s);
% Clean up.
clear cam;
release(videoPlayer);
release(pointTracker);
release(bodyDetector);