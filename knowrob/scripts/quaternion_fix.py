
import sys

s_in = sys.argv[1]

lev0 = s_in.split('>')
lev1 = lev0[1].split('<')

nums    = lev1[0].split(' ')
lev1[0] = ' '.join([nums[1],nums[2],nums[3],nums[0]])
lev0[1] = '<'.join(lev1)

s_out = '>'.join(lev0)

print(s_out)

