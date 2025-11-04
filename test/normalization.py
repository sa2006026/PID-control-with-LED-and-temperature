import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

def min_max_normalize(data):
    """
    Normalize data to range [0,1]
    """
    min_val = np.min(data)
    max_val = np.max(data)
    normalized = (data - min_val) / (max_val - min_val)
    return normalized

def z_score_normalize(data):
    """
    Normalize data using z-score (mean=0, std=1)
    """
    mean = np.mean(data)
    std = np.std(data)
    normalized = (data - mean) / std
    return normalized

def main():
    try:
        # Print current working directory and check if file exists
        print("Current working directory:", os.getcwd())
        file_path = 'test/data.txt'
        print("Checking if file exists:", os.path.exists(file_path))
        
        # Read data using a different method
        with open(file_path, 'r') as file:
            data = np.array([float(line.strip()) for line in file if line.strip()])
        
        print(f"Successfully loaded {len(data)} data points")
        
        # Perform normalizations
        min_max_normalized = min_max_normalize(data)
        z_score_normalized = z_score_normalize(data)
        
        # Create DataFrame with results
        results = pd.DataFrame({
            'Original': data,
            'Min-Max Normalized': min_max_normalized,
            'Z-Score Normalized': z_score_normalized
        })
        
        # Save results
        results.to_csv('./test/normalized_data.txt', index=False)
        
        # Create visualization
        plt.figure(figsize=(12, 6))
        
        plt.subplot(1, 2, 1)
        plt.plot(data, label='Original', color='blue')
        plt.plot(min_max_normalized, label='Min-Max', color='red')
        plt.title('Original vs Min-Max Normalized')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(1, 2, 2)
        plt.plot(data, label='Original', color='blue')
        plt.plot(z_score_normalized, label='Z-Score', color='green')
        plt.title('Original vs Z-Score Normalized')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig('test/normalization_plot.png')
        plt.close()
        
        # Print summary statistics
        print("\nOriginal Data Statistics:")
        print(f"Min: {data.min():.2f}")
        print(f"Max: {data.max():.2f}")
        print(f"Mean: {data.mean():.2f}")
        print(f"Std: {data.std():.2f}")
        
        print("\nNormalized Data Ranges:")
        print("Min-Max Normalization:")
        print(f"Min: {min_max_normalized.min():.2f}")
        print(f"Max: {min_max_normalized.max():.2f}")
        
        print("\nZ-Score Normalization:")
        print(f"Mean: {z_score_normalized.mean():.2f}")
        print(f"Std: {z_score_normalized.std():.2f}")
        
    except Exception as e:
        print(f"Error: {e}")
        print("Error location:", e.__traceback__.tb_lineno)

if __name__ == "__main__":
    main()
