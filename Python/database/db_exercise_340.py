"""
데이터베이스 연습 #340
SQLite CRUD 실습
"""
import sqlite3

def setup_340():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_340 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_340_0', 984.06, 'A'), ('item_340_1', 520.83, 'C'), ('item_340_2', 168.53, 'C'), ('item_340_3', 432.11, 'B'), ('item_340_4', 239.54, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_340 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_340()
    cursor = conn.cursor()

    print(f"DB 연습 #340")
    cursor.execute('SELECT * FROM items_340')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_340 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
