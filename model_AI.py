import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.linear_model import LinearRegression
import syft as sy  # Untuk Federated Learning

# 1. Linear Regression
X = np.array([[1], [2], [3], [4], [5]])
y = np.array([2, 4, 6, 8, 10])
lin_reg = LinearRegression().fit(X, y)
print("Linear Regression Coef:", lin_reg.coef_)

# 2. Two-Layer Neural Network
class TwoLayerNN(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):  # Perbaikan: Gunakan __init__
        super(TwoLayerNN, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(hidden_size, output_size)
    
    def forward(self, x):
        x = self.fc1(x)
        x = self.relu(x)
        x = self.fc2(x)
        return x

model = TwoLayerNN(1, 5, 1)  # Sekarang seharusnya tidak error

# 3. Optimisation (SGD, Adam, RMSprop)
optimizer_sgd = optim.SGD(model.parameters(), lr=0.01)
optimizer_adam = optim.Adam(model.parameters(), lr=0.01)
optimizer_rmsprop = optim.RMSprop(model.parameters(), lr=0.01)

# 4. AI in Privacy - Federated Learninga
from syft.frameworks.torch import TorchHook  # Impor TorchHook dari modul yang benar
from syft.workers.virtual import VirtualWorker  # Impor VirtualWorker

hook = TorchHook(torch)  # Inisialisasi hook dengan TorchHook yang benar
virtual_worker_1 = VirtualWorker(hook, id="worker1")  # Gunakan kelas VirtualWorker yang diimpor
virtual_worker_2 = VirtualWorker(hook, id="worker2")

print("Federated Learning environment setup done.")