import socket

HOST = '127.0.0.1'
PORT = 5001

def get_cognex_data():
    """
    Function to get tool data from the socket connection.
    Returns a dictionary with tool data.
    """

    board_list = []

    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        data = s.recv(4096).decode('ascii').strip()
        #print(f"Datos recibidos: {data}")
        
        # Suponiendo que los datos tienen el formato 'x=45.4522,y=149.082'
        parts = data.split(',')
        
        #s.close()
    
    #parts = ['', '', '8416.00', '', '10675.00', '', '', '', '7375.00', '']
    print(parts)

    for i in parts:
        if i == '':
            board_list.append(' ')
        elif float(i) < 9500:
            board_list.append('X')
        else:
            board_list.append('O')
        

    return board_list
