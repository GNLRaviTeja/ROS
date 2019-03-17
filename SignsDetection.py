
# coding: utf-8

# In[1]:


# Part 3 - Making new predictions
import numpy as np
from keras.preprocessing import image
from keras.models import Sequential, load_model

def detect_signs(sign_image):
    classifier = load_model('model6.h5')
    test_image = image.load_img(sign_image, target_size = (64, 64))
    test_image = image.img_to_array(test_image)
    test_image = np.expand_dims(test_image, axis = 0)
    pred = classifier.predict(test_image)
    result = pred[0]
    answer = np.argmax(result)
    if answer == 0:
        return("left")
    elif answer == 1:
        return("misc")
    elif answer == 2:
        return("right")
    elif answer == 3:
        return("stop")
    elif answer == 4:
        return("traffic_light")
    elif answer == 5:
        return("u")

