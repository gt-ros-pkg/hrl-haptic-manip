import os
import sys

if __name__ == '__main__':
    base = '/home/mkillpack'

    os.system('unzip '+base+'/Desktop/cvxgen.zip')
    os.system('mkdir '+base+'/Desktop/'+sys.argv[1])
    file_list = ['ldl', 'util', 'solver', 'matrix_support']
    for f in file_list:
        text = None

        with open('./cvxgen/'+f+'.c') as inp:
            text = inp.readlines()
        inp.close()
        
        ind = text.index('#include "solver.h"\n')
        text[ind] = '#include "'+sys.argv[1]+'.h"\n'
        f_out = open(base+'/Desktop/'+sys.argv[1]+'/'+f+'.cpp', 'w+')
        f_out.writelines(text)
        f_out.close()

    os.system('cp ./cvxgen/solver.h '+base+'/Desktop/'+sys.argv[1]+'/'+sys.argv[1]+'.h')
    os.system('rm -rf cvxgen')
    
