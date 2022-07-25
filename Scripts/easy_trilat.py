from easy_trilateration.model import *  
from easy_trilateration.least_squares import easy_least_squares  
from easy_trilateration.graph import *  
  
if __name__ == '__main__':  
    arr = [#Circle(0.0, -50, 800), #c0
           Circle(0.0, -50, 800), #c0 again
           Circle(0.0, 50, 800), #c1
           Circle(-50, 0.0, 848)] #c2
    draw(arr)
    result, meta = easy_least_squares(arr)  
    create_circle(result, target=True)  
    #draw(arr)
    #draw(result)
