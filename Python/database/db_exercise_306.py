"""
데이터베이스 연습 #306
SQLite CRUD 실습
"""
import sqlite3

def setup_306():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_306 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_306_0', 47.09, 'B'), ('item_306_1', 572.28, 'A'), ('item_306_2', 929.17, 'B'), ('item_306_3', 291.72, 'C'), ('item_306_4', 547.87, 'B'), ('item_306_5', 890.37, 'A')]
    ]
    cursor.executemany(
        'INSERT INTO items_306 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_306()
    cursor = conn.cursor()

    print(f"DB 연습 #306")
    cursor.execute('SELECT * FROM items_306')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_306 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
