import os
import openpyxl

def read_excel(filename):
    # Ermittle den vollständigen Dateipfad zur Excel-Datei
    full_path = os.path.join(os.path.dirname(__file__), filename)
    
    # Öffne die Excel-Datei
    wb = openpyxl.load_workbook(full_path)
    ws = wb.active

    # Liste zum Speichern der schwarzen Zellen
    black_cells = []

    # Iteriere über jede Zelle im Excel-Arbeitsblatt
    for row in range(2, 82):  # Annahme: 80x80 Zellen
        for column in range(2, 82):
            cell = ws.cell(row=row, column=column)
            if cell.fill.start_color.index != '00000000':  # Farbcode für schwarze Füllung
                #Speichere die Koordinaten der schwarzen Zellen
                black_cells.append([((column*10)-20, (row*10) - 20), ((column*10)-10, (row*10)-20), ((column*10)-10, (row*10)-10), ((column*10)-20, (row*10)-10)])

    # Schließe die Excel-Datei
    wb.close()

    return black_cells

def write_output(output_filename, black_cells):
    # Ermittle den vollständigen Dateipfad zur Ausgabedatei
    full_output_path = os.path.join(os.path.dirname(__file__), output_filename)

    # Schreibe die schwarzen Zellen in die Ausgabedatei
    with open(full_output_path, 'w') as f:
        f.write('OBSTACLES = ' + str(black_cells))

# Hauptprogramm
if __name__ == "__main__":
    excel_filename = 'map.xlsx'  # Passe den Dateinamen an
    output_filename = 'obstacles.py'  # Passe den Ausgabedateinamen an

    black_cells = read_excel(excel_filename)
    write_output(output_filename, black_cells)