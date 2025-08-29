"""
데이터베이스 연습 #346
SQLite CRUD 실습
"""
import sqlite3

def setup_346():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_346 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_346_0', 768.26, 'C'), ('item_346_1', 515.7, 'C'), ('item_346_2', 123.06, 'C'), ('item_346_3', 825.05, 'B'), ('item_346_4', 546.62, 'A'), ('item_346_5', 762.4, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_346 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_346()
    cursor = conn.cursor()

    print(f"DB 연습 #346")
    cursor.execute('SELECT * FROM items_346')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_346 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
