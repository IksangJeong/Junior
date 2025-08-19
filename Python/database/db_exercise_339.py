"""
데이터베이스 연습 #339
SQLite CRUD 실습
"""
import sqlite3

def setup_339():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_339 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_339_0', 833.0, 'C'), ('item_339_1', 150.26, 'B'), ('item_339_2', 349.64, 'A'), ('item_339_3', 525.9, 'B'), ('item_339_4', 481.4, 'B'), ('item_339_5', 724.77, 'B'), ('item_339_6', 325.72, 'B'), ('item_339_7', 54.24, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_339 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_339()
    cursor = conn.cursor()

    print(f"DB 연습 #339")
    cursor.execute('SELECT * FROM items_339')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_339 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
