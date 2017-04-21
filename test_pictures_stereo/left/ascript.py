from os import rename, listdir

badprefix = "left"
fnames = listdir('.')

for fname in fnames:
    #print fname[5:9]
    #print fname[6:9]
    print fname[0:4]
    if( fname[0:4]==badprefix):
        number= str(fname[4:8])
        print number
        count = int(number)
        print count-64
        rename(fname, "./test/"+badprefix+str(count-64))
