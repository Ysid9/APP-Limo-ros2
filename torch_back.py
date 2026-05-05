import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import math
import random

class PioneerNN(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(PioneerNN, self).__init__()
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size

        # Définir les couches
        self.hidden = nn.Linear(input_size, hidden_size)
        self.output = nn.Linear(hidden_size, output_size)

        # Fonction d'activation (tanh comme dans l'implémentation originale)
        self.activation = nn.Tanh()
	
	# Initialisation Xavier : calibre la variance selon fan_in/fan_out
        nn.init.xavier_uniform_(self.hidden.weight)
        nn.init.xavier_uniform_(self.output.weight)
        
       	#  Initialisation des poids avec une distribution uniforme [-1, 1]
        #nn.init.uniform_(self.hidden.weight, -1.0, 1.0)
        #nn.init.uniform_(self.output.weight, -1.0, 1.0)
        
        nn.init.zeros_(self.hidden.bias)
        nn.init.zeros_(self.output.bias)

    def forward(self, x):
        if not isinstance(x, torch.Tensor):
            x = torch.tensor(x, dtype=torch.float32)

        x = self.activation(self.hidden(x))
        x = self.activation(self.output(x))
        return x

    def run_nn(self, inputs):
        """Méthode compatible avec l'interface originale"""
        with torch.no_grad():
            x = torch.tensor(inputs, dtype=torch.float32)
            outputs = self.forward(x)
            return outputs.tolist()

    def load_weights_from_json(self, json_obj):
        """Charger les poids à partir d'un objet JSON (format original)"""
        meta_input  = json_obj.get("input_size")
        meta_hidden = json_obj.get("hidden_size")
        meta_output = json_obj.get("output_size")
        if meta_input is not None and meta_input != self.input_size:
            raise ValueError(f"Le fichier a été sauvegardé avec input_size={meta_input}, mais ce réseau a input_size={self.input_size}")
        if meta_hidden is not None and meta_hidden != self.hidden_size:
            raise ValueError(f"Le fichier a été sauvegardé avec hidden_size={meta_hidden}, mais ce réseau a hidden_size={self.hidden_size}")
        if meta_output is not None and meta_output != self.output_size:
            raise ValueError(f"Le fichier a été sauvegardé avec output_size={meta_output}, mais ce réseau a output_size={self.output_size}")

        input_weights = torch.zeros(self.input_size, self.hidden_size)
        for i in range(self.input_size):
            for j in range(self.hidden_size):
                input_weights[i][j] = json_obj["input_weights"][i][j]

        output_weights = torch.zeros(self.hidden_size, self.output_size)
        for i in range(self.hidden_size):
            for j in range(self.output_size):
                output_weights[i][j] = json_obj["output_weights"][i][j]

        with torch.no_grad():
            self.hidden.weight.copy_(input_weights.t())
            self.output.weight.copy_(output_weights.t())

    def save_weights_to_json(self):
        """Convertir les poids en format JSON compatible avec l'original"""
        input_weights = []
        hidden_weights = self.hidden.weight.detach().t()
        for i in range(self.input_size):
            row = []
            for j in range(self.hidden_size):
                row.append(float(hidden_weights[i][j]))
            input_weights.append(row)

        output_weights = []
        out_weights = self.output.weight.detach().t()
        for i in range(self.hidden_size):
            row = []
            for j in range(self.output_size):
                row.append(float(out_weights[i][j]))
            output_weights.append(row)

        return {
            "input_size": self.input_size,
            "hidden_size": self.hidden_size,
            "output_size": self.output_size,
            "input_weights": input_weights,
            "output_weights": output_weights,
        }
