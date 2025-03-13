print(3.14 * 10 * 10)
print(3.14 * 10**2)
print (type(10))
print (type(3.14))
print (type("python"))

r = 20
PI = 3.14
area = PI * r**2
print (area)

lst = [10, 20, 30,40,50]
print (lst)
print (lst[2])
lst[2] = 90
print (lst)

print (len(lst))
print (lst[0:3])
print (lst[2:])
print (lst[:3])
print (lst[:-1])

car = {'HP': 200,'make': "BNW"}
print (car['HP'])

car['color'] = "white"
print (car)

temp = -10
if temp < 0:
    print ("영하입니다.")
else:
    print ("영상입니다.")