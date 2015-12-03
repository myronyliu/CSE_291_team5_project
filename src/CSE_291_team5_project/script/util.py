def scaled_rect_coords(px, py, pw, ph, nCols,nRows):
    xMid =  2.0*(px + pw/2.0)/nRows - 1.0
    yMid = -2.0*(py + ph/2.0)/nCols + 1.0
    return (xMid, yMid, (pw+0.0) / nCols, (ph+0.0) / nRows)
