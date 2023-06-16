import numpy as np

linesensor=np.load('unused_files/matrix_data.npy')
print(linesensor)

# see all differences between sifferent between the first vector and all others


first_array = linesensor[0]  # Get the first array
# Calculate the difference between the first array and all the other arrays
diff = linesensor[1:] - first_array

# checking all values that arent 0
differences = []
for i, row in enumerate(diff):
    indices = [j for j, val in enumerate(row) if val != 0]
    differences.append(indices)
#remove from nonzero_indices all the empty lists

# print(differences)

# #if any value in differences[1]==0, then something on the right, if any value in differences[1]==6, then something on the left and print line number


for i, row in enumerate(differences):
    if 0 in row:
        print("something on the right on line", i+1)
    if 6 in row:
        print("something on the left on line", i+1)