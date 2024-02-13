import tensorflow as tf
config = tf.ConfigProto()
config.gpu_options.allow_growth = True

# dowloading the dataset
#with tf.device("/device:GPU:0"):
(x_train, y_train), (x_test, y_test) = tf.keras.datasets.mnist.load_data()

# plot an example
import matplotlib.pyplot as plt
image_index= 7777  # You may select anything up to 60,000
print("Label of image at index", image_index, "->", y_train[image_index])     # The label is 8
plt.imshow(x_train[image_index], cmap='Greys')                                # add the image to the plot
# plt.show()                                                                  # show the plot
print("--------------")
print("    [dataset]: dataset shape->", x_train.shape)


# Reshaping the array to 4-dims so that it can work with the Keras API
x_train = x_train.reshape(x_train.shape[0], 28, 28, 1)
x_test = x_test.reshape(x_test.shape[0], 28, 28, 1)
input_shape = (28, 28, 1)
# Making sure that the values are float so that we can get decimal points after division
x_train = x_train.astype('float32')
x_test = x_test.astype('float32')
# Normalizing the RGB codes by dividing it to the max RGB value.
x_train = x_train/255.0
x_test = x_test/255.0
print('    [dataset]: training_set shape:', x_train.shape)
print('    [dataset]: Number of images in x_train->', x_train.shape[0])
print('    [dataset]: Number of images in x_test->', x_test.shape[0])
print("--------------")


# Importing the required Keras modules containing model and layers
from keras.models import Sequential
from keras.layers import Dense, Conv2D, Dropout, Flatten, MaxPooling2D

print("########### TRAINING ############")
# Creating a Sequential Model and adding the layers
model = tf.keras.models.Sequential([
    tf.keras.layers.Conv2D(28, kernel_size=(3,3), input_shape=input_shape),    # convolultional layer
    tf.keras.layers.MaxPooling2D(pool_size=(2, 2)),                            # Max Poolong layer
    tf.keras.layers.Flatten(),                                                 # flatten the input, from many to one dimension
    #tf.keras.layers.Dense(512, activation=tf.nn.relu),                         # NN dense layer (fc), with 512 output
    #tf.keras.layers.Dropout(rate=0.2),                                         # to avoid overfitting sets 'rate' fraction of input to 0
    tf.keras.layers.Dense(128, activation=tf.nn.relu),                         # NN dense layer (fc), with 128 output
    tf.keras.layers.Dropout(rate=0.2),                                         # to avoid overfitting sets 'rate' fraction of input to 0
    tf.keras.layers.Dense(10, activation=tf.nn.softmax)                        # NN dense layer (fc), with 10 output, the 10 classes of mnist
])


# compiling and fitting the model
model.compile(optimizer='adam', 
              loss='sparse_categorical_crossentropy', 
              metrics=['accuracy'])
model.fit(x=x_train,y=y_train, epochs=5)

# testing the model
print("[testing]: Evaluation results(loss, accuracy):", model.evaluate(x_test, y_test))
print("--------------")


import numpy as np
image_index = np.random.randint(10000)
img_rows, img_cols= 28, 28
plt.imshow(x_test[image_index].reshape(28, 28),cmap='Greys')
pred = model.predict(x_test[image_index].reshape(1, img_rows, img_cols, 1))
print("[testing]: Predicted value for image at index", image_index, "->", pred.argmax())
plt.show()

'''
(x_train, y_train), (x_test, y_test) = tf.keras.datasets.mnist.load_data()

import numpy as np
from sklearn.linear_model import SGDClassifier

print("dataset sizes: X_train->", x_train.shape, "y_train->", y_train.shape)
nsamples= x_train.shape[0]
d= x_train.shape[1]*x_train.shape[1]
x_train_2 = x_train.reshape(nsamples,d)

idx= np.random.randint(60000)
some_digit = x_train_2[idx]

sgd_clf = SGDClassifier(max_iter=5, random_state=42)
sgd_clf.fit(x_train_2, y_train)
print("Possible classes: ", sgd_clf.classes_)
print("Expected value->", y_train[idx])
print("Predicted value for image at index", idx, "->", sgd_clf.predict([some_digit]))

some_digit_scores = sgd_clf.decision_function([some_digit])
print(some_digit_scores)

print("best match:", np.argmax(some_digit_scores))
'''
