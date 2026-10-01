import numpy as np
import cv2

##### VIDEO ADD TEXT FUNCTION (adds text to video frame)
def WriteVideoText(rgba, textflag: bool, t: float):
    (height, width, _) = np.shape(rgba)

    # target state text
    if textflag: # if flag is true
        text = 'NO!'
        text_color = (255, 0, 0) # red text
    else:
        text = 'YES!'
        text_color = (0, 255, 0) # green text
    
    text_coords = (10, 30)
    text_font = cv2.FONT_HERSHEY_SIMPLEX
    text_font_scale = 1.0
    text_thickness = 2
    text_line_type = cv2.LINE_AA

    cv2.putText(
        img=rgba, text=text, org=text_coords, fontFace=text_font, fontScale=text_font_scale, color=text_color, thickness=text_thickness, lineType=text_line_type
    )

    # time text
    text = f'{t:.3f}'
    text_color = (0, 0, 0) # black text
    
    text_coords = (width - 80, 30)
    text_font = cv2.FONT_HERSHEY_SIMPLEX
    text_font_scale = 0.8
    text_thickness = 2
    text_line_type = cv2.LINE_AA

    cv2.putText(
        img=rgba, text=text, org=text_coords, fontFace=text_font, fontScale=text_font_scale, color=text_color, thickness=text_thickness, lineType=text_line_type
    )

    # return new image
    return rgba

##### VIDEO ADD POINT FUNCTION (adds point at coords to video frame)
def WriteVideoPoint(rgba, coords: tuple):
    if coords[0] == None: return rgba # if coords not in image

    # target state point
    u = int(coords[0])
    v = int(coords[1])

    point_radius = 15 # (px)
    point_color = (0, 0, 0) # black point
    point_thickness = -1 # filled point
    point_line_type = cv2.LINE_AA

    cv2.circle(
        img=rgba, center=(u, v), radius=point_radius, color=point_color, thickness=point_thickness, lineType=point_line_type 
    )
    # return new image
    return rgba


