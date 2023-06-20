


#read file
with open("ciberRatoTools/simulator/planning.out", "r") as file:
    map_data = file.read()

# Clean up the map by removing whitespace and splitting it into lines
map_data = map_data.split('\n')

# add '+' to when theres two '-' in betwen a empty space
for i in range(len(map_data)):
    for j in range(len(map_data[i])):
        if map_data[i][j] == '-':
            if map_data[i][j+2] == '-':
                map_data[i:j+1]='+'



# print the new map
for i in range(0, 21):
    for j in range(0, 49):
        print(map_data[i][j], end='')
    print('\n', end='')

       
