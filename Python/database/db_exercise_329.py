"""
데이터베이스 연습 #329
SQLite CRUD 실습
"""
import sqlite3

def setup_329():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_329 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_329_0', 106.71, 'A'), ('item_329_1', 840.29, 'B'), ('item_329_2', 609.12, 'C'), ('item_329_3', 955.31, 'B'), ('item_329_4', 969.23, 'B'), ('item_329_5', 39.33, 'B'), ('item_329_6', 785.71, 'B'), ('item_329_7', 495.7, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_329 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_329()
    cursor = conn.cursor()

    print(f"DB 연습 #329")
    cursor.execute('SELECT * FROM items_329')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_329 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
