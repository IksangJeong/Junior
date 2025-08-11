"""
데이터베이스 연습 #325
SQLite CRUD 실습
"""
import sqlite3

def setup_325():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_325 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_325_0', 918.87, 'C'), ('item_325_1', 157.41, 'A'), ('item_325_2', 623.58, 'B'), ('item_325_3', 817.06, 'B'), ('item_325_4', 796.99, 'A')]
    ]
    cursor.executemany(
        'INSERT INTO items_325 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_325()
    cursor = conn.cursor()

    print(f"DB 연습 #325")
    cursor.execute('SELECT * FROM items_325')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_325 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
