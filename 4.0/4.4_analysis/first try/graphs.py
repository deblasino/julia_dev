import pandas as pd
import matplotlib.pyplot as plt
import math

# 1. Caricamento dei dati
file_path = 'vmc_simulation_results.txt'
# Se il file ha spazi misti, usa delim_whitespace=True, altrimenti sep='\t'
try:
    df = pd.read_csv(file_path, sep='\t')
except:
    df = pd.read_csv(file_path, delim_whitespace=True)

# 2. Configurazione dei grafici
x_col = 'L_Segment'
metrics = [col for col in df.columns if col != x_col]

num_plots = len(metrics)
cols = 2
rows = math.ceil(num_plots / cols)

# MODIFICA 1: Aumentiamo l'altezza per ogni riga (da 4 a 6 pollici per riga)
# figsize=(larghezza, altezza)
fig, axes = plt.subplots(rows, cols, figsize=(16, 6 * rows))
axes = axes.flatten()

# 3. Generazione dei grafici
for i, metric in enumerate(metrics):
    ax = axes[i]
    ax.plot(df[x_col], df[metric], marker='o', linestyle='-', color='tab:blue')
    
    # Titoli ed etichette con font un po' più grandi per leggibilità
    
    ax.set_xlabel(x_col, fontsize=10)
    ax.set_ylabel(metric, fontsize=10)
    ax.grid(True, linestyle='--', alpha=0.7)

# Rimuovere eventuali sottografici vuoti
for j in range(i + 1, len(axes)):
    fig.delaxes(axes[j])

# MODIFICA 2: Aggiungiamo spazio esplicito tra i grafici
# hspace = spazio verticale (height space), wspace = spazio orizzontale (width space)
plt.subplots_adjust(hspace=0.5, wspace=0.3)

# Un'alternativa a subplots_adjust è tight_layout con padding extra, 
# ma subplots_adjust dà più controllo manuale.
# plt.tight_layout(pad=4.0) 

print("Generazione grafici completata. File salvato come 'grafici_metriche_spaziati.png'")
plt.savefig('grafici_metriche_spaziati.png')
# plt.show() # Togli il commento se vuoi vederli a schermo
