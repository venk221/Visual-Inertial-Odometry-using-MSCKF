import pandas as pd

df=pd.read_csv('data.csv')

print(df.columns)

df=df[["#timestamp"," p_RS_R_x [m]"," p_RS_R_y [m]"," p_RS_R_z [m]"," q_RS_x []"," q_RS_y []"," q_RS_z []"," q_RS_w []"]]

print(df.columns)
print(df.head())


file='groundtruth.txt'

f=open(file,'a')
    

for i, row in df.iterrows():

    f.write(str(row[0]))
    f.write(' ')
    f.write(str(row[1]))
    f.write(' ')
    f.write(str(row[2]))
    f.write(' ')
    f.write(str(row[3]))
    f.write(' ')
    f.write(str(row[4]))
    f.write(' ')
    f.write(str(row[5]))
    f.write(' ')
    f.write(str(row[6]))
    f.write(' ')
    f.write(str(row[7]))
    f.write(' ')
    f.write('\n')
    





f.close()
