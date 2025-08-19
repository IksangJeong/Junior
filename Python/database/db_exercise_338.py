"""
데이터베이스 연습 #338
SQLite CRUD 실습
"""
import sqlite3

def setup_338():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_338 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_338_0', 20.44, 'B'), ('item_338_1', 229.78, 'C'), ('item_338_2', 523.61, 'C'), ('item_338_3', 690.79, 'A'), ('item_338_4', 451.32, 'B'), ('item_338_5', 468.43, 'C'), ('item_338_6', 592.7, 'C'), ('item_338_7', 159.24, 'A')]
    ]
    cursor.executemany(
        'INSERT INTO items_338 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_338()
    cursor = conn.cursor()

    print(f"DB 연습 #338")
    cursor.execute('SELECT * FROM items_338')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_338 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
