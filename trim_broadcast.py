from __future__ import print_function
from sys import argv

def trim_broadcast( str ):
	with open(str) as fp:
		f = open (str+'.out', 'w')
		key = ""
		for line in fp:
			if len(line) > 10:
				temp = line.split ( )[0]
				if not key == temp:
					f.write(line)
					key = temp
		f.close()
	return;

total = len(argv)
cmdargs = str(argv)

for i in range (1, total):
	trim_broadcast(argv[i]);