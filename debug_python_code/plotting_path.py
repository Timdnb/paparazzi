import csv
import matplotlib.pyplot as plt

def plot_csv_data(csv_file):
    # Lists to store x and y positions
    x_positions = []
    y_positions = []

    # Read data from CSV file
    with open(csv_file, 'r') as file:
        csv_reader = csv.reader(file)
        
        next(csv_reader)  # Skip header if exists
        next(csv_reader)
        for i, row in enumerate(csv_reader):
            if i % 2:
                continue
            # Extract x and y positions from 2nd and 3rd columns
            x = float(row[1])  # Assuming 2nd column contains x position
            y = float(row[2])  # Assuming 3rd column contains y position
            x_positions.append(x)
            y_positions.append(y)

    print(i)

    # Plot the data
    plt.figure(figsize=(8, 6))
    plt.scatter(x_positions, y_positions, color='blue', marker='o', alpha=0.5)
    plt.title('Plot of X vs Y Positions')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.grid(True)
    plt.show()

# Example usage: Provide the path to your CSV file
csv_file_path = '20240322-120422-cornerdata.csv'
plot_csv_data(csv_file_path)
