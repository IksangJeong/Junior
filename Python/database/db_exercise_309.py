"""
데이터베이스 연습 #309
SQLite CRUD 실습
"""
import sqlite3

def setup_309():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_309 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_309_0', 411.36, 'C'), ('item_309_1', 923.87, 'C'), ('item_309_2', 772.14, 'C'), ('item_309_3', 126.24, 'B'), ('item_309_4', 373.87, 'A'), ('item_309_5', 226.94, 'A'), ('item_309_6', 483.4, 'B'), ('item_309_7', 750.26, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_309 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_309()
    cursor = conn.cursor()

    print(f"DB 연습 #309")
    cursor.execute('SELECT * FROM items_309')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_309 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
