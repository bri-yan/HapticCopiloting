def write(target):
    command = 'w' + str(target).zfill(6)
    print(f'Sent command: {command}')

write(-10)