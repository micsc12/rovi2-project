from os import rename, listdir

badprefix = "right"
fnames = listdir('.')

for fname in fnames:
    #print fname[5:9]
    #print fname[6:9]
    #print fname[0:5]
    if( fname[0:5]==badprefix):
        number= str(fname[5:9])
        print number
        count = int(number)
        print count-345
        rename(fname, "./test/"+badprefix+str(count-345))
