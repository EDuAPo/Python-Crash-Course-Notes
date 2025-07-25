available_toppings = ['mushrooms', 'olives', 'green peppers','pepperoni', 'pineapple', 'extra cheese']    # 处定义了一个列表，其中包含比萨店供应的配料

requested_toppings = ['mushrooms', 'french fries', 'extra cheese']                                        # 定义了一个列表，其中包含创建顾客点的配料

for requested_topping in requested_toppings:
    if requested_topping in available_toppings:
        print(f"Adding {requested_topping}.")
    else:
        print(f"Sorry, we don't have {requested_topping}.")
        
print("\nFinished making your pizza!")






requested_toppings = []

if requested_toppings:
  for requested_topping in requested_toppings:
      print(f"Adding {requested_topping}.")
      print("\nFinished making your pizza!")
else:
      print("Are you sure you want a plain pizza?")