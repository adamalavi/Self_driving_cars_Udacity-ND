import os
import csv
import math

samples = []
with open('/home/workspace/CarND-Behavioral-Cloning-P3/data/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        samples.append(line)

from sklearn.model_selection import train_test_split
train_samples, validation_samples = train_test_split(samples, test_size=0.2)

import cv2
import numpy as np
import sklearn
from sklearn.utils import shuffle

def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []
            for batch_sample in batch_samples:
                for i in range(3):
                    name = '/home/workspace/CarND-Behavioral-Cloning-P3/data/IMG/'+batch_sample[i].split('/')[-1]
                    image = cv2.cvtColor(cv2.imread(name), cv2.COLOR_BGR2RGB)
                    center_angle = float(batch_sample[3])
                    images.append(image)
                    if(i==0):
                        angles.append(center_angle)
                    elif(i==1):
                        angles.append(center_angle+0.2)
                    elif(i==2):
                        angles.append(center_angle-0.2)
                    # Augmentation
                    images.append(cv2.flip(image,1))
                    if(i==0):
                        angles.append(center_angle*-1)
                    elif(i==1):
                        angles.append((center_angle+0.2)*-1)
                    elif(i==2):
                        angles.append((center_angle-0.2)*-1)

            X_train = np.array(images)
            y_train = np.array(angles)
            yield sklearn.utils.shuffle(X_train, y_train)

# Set our batch size
batch_size=32

# compile and train the model using the generator function
train_generator = generator(train_samples, batch_size=batch_size)
validation_generator = generator(validation_samples, batch_size=batch_size)


from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Cropping2D, Dropout
from keras.layers.convolutional import Convolution2D
model = Sequential()
# Preprocess incoming data, centered around zero with small standard deviation 
model.add(Lambda(lambda x: x/127.5 - 1.,
        input_shape=(160, 320, 3),
        output_shape=(160, 320, 3)))
# Cropping
model.add(Cropping2D(cropping = ((70, 25),(0, 0))))
# Convolutional layer 1
model.add(Convolution2D(24, (5, 5), strides=(2, 2), activation='relu'))
# Convolutional layer 2
model.add(Convolution2D(36, (5, 5), strides=(2, 2), activation='relu'))
# Convolutional layer 3
model.add(Convolution2D(48, (5, 5), strides=(2, 2), activation='relu'))
# Convolutional layer 4
model.add(Convolution2D(64, (3, 3), activation='relu'))
# Convolutional layer 5
model.add(Convolution2D(64, (3, 3), activation='relu'))
# Flatten
model.add(Flatten())
# Fully connected layer 1
model.add(Dense(100))
# Dropout to avoid overfitting
model.add(Dropout(0.25))
# Fully connected layer 2
model.add(Dense(50))
model.add(Dense(1))

model.compile(loss='mse', optimizer='adam')
model.fit_generator(train_generator, \
            steps_per_epoch=math.ceil(len(train_samples)/batch_size), \
            validation_data=validation_generator, \
            validation_steps=math.ceil(len(validation_samples)/batch_size), \
            epochs=10, verbose=1)
model.save('model.h5')
print("Model saved!")