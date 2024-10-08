from copy import deepcopy
# MaterialList = ["bread", "meat", "cheeze", "pickle", "onion", "sauce", "tomato", "lettuce"]

class Task:
    def __init__(self) -> None:
        self.templete = [{'mode': "init_pos",    'material': -1},
                         {'mode': "vision",      'material': -1},
                         {'mode': "pnp",         'material': -1},
                         {'mode': "tool_return", 'material': -1}]

    def make_task(self, mode, material):
        task = deepcopy(self.templete[mode])
        task['material'] = material
        # task['material'] = MaterialList[material]
        return task

    def order_to_task(self, order):
        tasks = [self.templete[0]]

        for material, repeat in enumerate(order):
            if material == 0:
                vision = self.make_task(1, material)
                tasks.append(vision)

                pnp = self.make_task(2, material)
                tasks.append(pnp)

            else:
                vision = self.make_task(1, material)
                pnp = self.make_task(2, material)

                task = [vision, pnp] * repeat
                
                tasks.extend(task)

            tool_return = self.make_task(3, material)
            tasks.append(tool_return)
            
        # bread
        vision = self.make_task(1, 0)
        tasks.append(vision)

        pnp = self.make_task(2, 0)
        tasks.append(pnp)

        tool_return = self.make_task(3, 0)
        tasks.append(tool_return)

        return tasks
    

def main():

    planner = Task()

    order = [2, 1, 1, 3, 1, 1, 1, 1]

    tasks = planner.order_to_task(order)

    for i in tasks:
        print(i)

if __name__ == '__main__':
    main()