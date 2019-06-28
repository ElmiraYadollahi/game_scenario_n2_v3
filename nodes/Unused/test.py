import random


for x in range(10):
  print random.randint(1,101)

print ("...")

for x in range(2):
	print random.randint(2,4)

print random.sample(range(100), 10)

order = random.sample(range(2,5), 2)
print order
#print order.sort(key=int)
print sorted(order, key=int)
