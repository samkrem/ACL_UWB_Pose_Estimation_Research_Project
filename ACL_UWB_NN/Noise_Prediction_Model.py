import numpy as np
import matplotlib.pyplot as plt

import torch
import torch.nn as nn
import torch.distributions as distributions
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset

from sklearn.model_selection import train_test_split
from pathlib import Path
from itertools import product
import pandas as pd
import io
#organize and prepare data
file_path = ".csv"
df = pd.read_csv(file_path)

csv_values= df.values
x_poses=csv_values[:, :6]
z_true=csv_values[:,6]
w_x=csv_values[:,7]
z_x=csv_values[:,8]

z_x=z_x.reshape(-1,1)
xz_input=np.hstack((x_poses, z_x))
w_x=w_x.reshape(-1,1)

xz_input=xz_input.astype(np.float32)
w_x = w_x.astype(np.float32)

#Run on GPU if possible
device = "cuda" if torch.cuda.is_available() else "cpu"

#Partition data into train, validation, and testing
xz_train, xz_temp, w_x_train, w_x_temp = train_test_split(xz_input, w_x, test_size=0.3, random_state=42)
xz_val, xz_test, w_x_val, w_x_test = train_test_split(xz_temp, w_x_temp, test_size=0.5, random_state=42)

xz_pose_train_scaled_tensor = torch.tensor(xz_train, dtype=torch.float32, device=device)
xz_pose_valid_scaled_tensor = torch.tensor(xz_val, dtype=torch.float32, device=device)
xz_pose_test_scaled_tensor = torch.tensor(xz_test, dtype=torch.float32, device=device)

w_x_train_tensor = torch.tensor(w_x_train, dtype=torch.float32, device=device)
w_x_valid_tensor = torch.tensor(w_x_val, dtype=torch.float32, device=device)
w_x_test_tensor = torch.tensor(w_x_test, dtype=torch.float32, device=device)

#Neural Network Model
class NoisePredictionModel(nn.Module):
    #in_features: UWB change in pose and true distance of two robots
    #Hidden Layer 1: # of neurons
    #Hidden Layer 2: neurons
    #Output layer: UWB Sensor Noise
    def __init__(self, in_features=7, h1=256, h2=128, h3=64, h4=32, out_features=1, dropout_rate=0.1):
        super().__init__()
        self.fc1 = nn.Linear(in_features, h1) #trial and error, adjust/look at hyperparameters
        self.dropout1 = nn.Dropout(p=dropout_rate)
        self.fc2 = nn.Linear(h1, h2)
        self.dropout2 = nn.Dropout(p=dropout_rate)
        self.fc3=nn.Linear(h2,h3)
        self.dropout3 = nn.Dropout(p=dropout_rate)
        self.fc4=nn.Linear(h3,h4)


        self.out = nn.Linear(h4, out_features)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = self.dropout1(x)
        x = torch.relu(self.fc2(x))
        x = self.dropout2(x)
        x=torch.relu(self.fc3(x))
        x=self.dropout3(x)
        x=torch.relu(self.fc4(x))
        x = self.out(x)
        return x
    
model = NoisePredictionModel()
model.to(device)


#Loss function and optimizer
loss_fn = nn.SmoothL1Loss()
optimizer = optim.AdamW(model.parameters(), lr=0.000004, weight_decay=0.04)

train_dataset = TensorDataset(xz_pose_train_scaled_tensor, w_x_train_tensor)
train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True)

epochs = 30
batch_size=64

epoch_count= []

w_x_train_loss_values= []

w_x_valid_loss_values = []

correctd_UWB_pose_test_loss_values = []

print_loss_rate=25


for epoch in range(epochs):
  #Set model to training mode
  model.train()
  for batch_xz_pose_scaled, batch_w_x in train_loader:
  #1. Forward pass
      batch_w_x_scaled_pred= model(batch_xz_pose_scaled) #predicted pose essentially? or is this the error
  #2. Calculate loss
  #WE MUST PUT THE UNSCALE THE PREDICTED DATA SO ITS IN THE ORIGNIAL SCALE OF THE OUTPUT SO OUR LOSS ISN'T FUCKED UP
      loss=loss_fn(batch_w_x_scaled_pred, batch_w_x)
  #3. Zero the gradient
      optimizer.zero_grad()
  #3. Perform backpropogation:
      loss.backward()
  #4. Step the optimizer
      optimizer.step()
  # Validation loop

  model.eval()
  with torch.inference_mode(): #testing the predictions
    w_x_valid_pred=model(xz_pose_valid_scaled_tensor)
    w_x_valid_loss=loss_fn(w_x_valid_pred, w_x_valid_tensor)

  if epoch%1==0: #just to print info
    epoch_count.append(epoch)
    w_x_valid_loss_values.append(w_x_valid_loss.cpu().detach().numpy())
    w_x_train_loss_values.append(loss.cpu().detach().numpy())
    print(f"Epoch:{epoch+1}| Loss: {loss} | Validation loss: {w_x_valid_loss}")

plt.figure()
plt.plot(epoch_count,w_x_valid_loss_values,"b", label="validation")
plt.plot(epoch_count,w_x_train_loss_values,"r", label="training")
plt.xlabel("Epochs")
plt.ylabel("Loss")
plt.title("Epochs vs Loss")
plt.legend()

model.eval()
with torch.inference_mode():
    # w_x_test_pred = model(xz_pose_test_scaled_tensor)
    w_x_test_pred = model(xz_pose_train_scaled_tensor)
    w_x_test_loss=loss_fn(w_x_test_pred, w_x_train_tensor) #TEST DATA
print(f"Test loss: {w_x_test_loss}")
print(w_x_test_pred)
#if test satisfactory
model.eval()
with torch.inference_mode():
    w_x_pred = model(torch.tensor(xz_input, dtype=torch.float32).to(device))
    w_x_pred= w_x_pred.cpu().numpy()

#Plot sample vs predicted mean
plt.plot(w_x, w_x_pred, 'o', label='Actual vs. Predicted')
plt.plot([min(w_x), max(w_x)], [min(w_x), max(w_x)], '--', label='Identity Line')

plt.xlabel("w(x) sampled")
plt.ylabel("w(x) predicted mean")
plt.title("w(x) predictions vs w(x) meausred ")
plt.legend()
plt.show()


print(w_x[0])
print(w_x_pred[0])
plt.hist(w_x)
plt.xlabel("Measured Noise")
plt.ylabel("Frequency")
plt.title("Frequency of Measured w(x)")

plt.hist(w_x_pred, color="red")
plt.xlabel("Predicted Noise")
plt.ylabel("Frequency")
plt.title("Frequency of Predicted w(x)")

#Display weight of each layer
parameters=model.state_dict()

weights = [parameters[key] for key in parameters if 'weight' in key]

biases = [parameters[key] for key in parameters if 'bias' in key]
for i in range(len(weights)):
  print(f"Layer {i+1} Weight: \n {weights[i].cpu().numpy()}")
for i in range(len(biases)):
  print(f"Layer {i+1} Bias: \n {biases[i].cpu().numpy()}")


#Saving model

# 1. Create models directory

MODEL_PATH = Path("~/Users/swarm/...")
MODEL_PATH.mkdir(parents=True, exist_ok=True)

# 2. Create model save path
MODEL_NAME="UWB_NN_data_model.pth" #pytorch saved as .pth /.pt
#
MODEL_SAVE_PATH= MODEL_PATH / MODEL_NAME

# 3. Save the model state dict
torch.save(obj=model.state_dict(), f=MODEL_SAVE_PATH)


# Create new instance of model and load saved state dict (make sure to put it on the target device)
loaded_model=NoisePredictionModel()
loaded_model.load_state_dict(torch.load(f=MODEL_SAVE_PATH))
loaded_model.to(device)
next(loaded_model.parameters()).device
